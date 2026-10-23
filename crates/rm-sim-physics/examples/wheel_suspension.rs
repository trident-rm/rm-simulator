// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Isolated, contact-free EGA-derived planar suspension bench, not a game model.
//! CAD dimensions: docs/suspension-recovery.md. Force law and load are provisional.
use glam::{DMat3, DQuat, DVec3};
use rm_sim_engine::{
    Pose,
    model::{
        BodyKind, BodyPolicy, BodyState, ColliderDescription, ContactMaterial, Inertial,
        JointDescription, JointKind, JointLimits, JointMotor, RigidBodyDescription,
        ShapeDescription,
    },
};
use rm_sim_physics::{
    StepDuration,
    cooking::CookedRigidBody,
    runtime::{BodyBinding, BodyKey, JointBinding, JointKey, PhysicalRuntime},
};

/// Provisional choices, not a recovered KJ85T33 manufacturer specification.
#[derive(Clone, Copy, Debug)]
pub struct Parameters {
    /// Damper extension stop / unloaded eye spacing, metres.
    pub free_length_m: f64,
    /// Total usable damper stroke, metres, independent of wheel displacement.
    pub stroke_m: f64,
    /// Linear compression stiffness, N/m.
    pub stiffness_n_m: f64,
    /// Symmetric viscous damping, N s/m; no seal friction model.
    pub damping_n_s_m: f64,
    /// Lumped moving test load, kg, including carrier/wheel equivalent.
    /// This is not the recovered mass of the real wheel module.
    pub load_kg: f64,
    /// Provisional original assembly mass, kg; internal split remains estimated.
    pub damper_mass_kg: f64,
}
impl Default for Parameters {
    fn default() -> Self {
        Self {
            free_length_m: 0.096,
            stroke_m: 0.030,
            stiffness_n_m: 3_000.,
            damping_n_s_m: 50.,
            load_kg: 5.,
            damper_mass_kg: 0.044,
        }
    }
}
impl Parameters {
    fn validate(self) -> Result<(), &'static str> {
        if ![
            self.free_length_m,
            self.stroke_m,
            self.stiffness_n_m,
            self.damping_n_s_m,
            self.load_kg,
            self.damper_mass_kg,
        ]
        .into_iter()
        .all(f64::is_finite)
            || self.stroke_m <= 0.
            || self.free_length_m <= self.stroke_m
            || self.stiffness_n_m <= 0.
            || self.damping_n_s_m < 0.
            || self.load_kg <= 0.
            || self.damper_mass_kg <= 0.
            || !(self.free_length_m - self.stroke_m..=self.free_length_m)
                .contains(&INITIAL_LENGTH_M)
        {
            return Err("invalid suspension bench parameters or initial length outside stops");
        }
        Ok(())
    }
}
/// Native circular mount fit eye spacing; one saved state, metres.
pub const INITIAL_LENGTH_M: f64 = 0.08755;
/// Two CAD rocker centers, 25 mm apart; corresponding pivots are 50 mm apart.
const ARM: [f64; 3] = [0.011754609365650348, 0., -0.022064205371165435];
const SECOND: [f64; 3] = [0.05, 0., 0.];
/// Native mounting geometry rotated into the bench X/Z plane. Bench +Z is an
/// explicit illustrative gravity orientation, not a certified robot-up transform.
const MOVING_EYE: [f64; 3] = [0.053754609365649485, -0.0014, -0.09031420537116566];
const FIXED_EYE: [f64; 3] = [-0.0335, -0.0014, -0.0975];
const ORIGIN: DVec3 = DVec3::new(0., 0., 0.3);
/// Independently STEP-checked mass of one source rocker, kg.
pub const ARM_MASS_KG: f64 = 0.014192920065877;
// Native principal tensor about COM [12.5,0,4] mm; local source products are zero.
// Independently checked against STEP in docs/suspension-recovery.md.
const ARM_INERTIA_KG_M2: [f64; 3] = [5.841358266e-7, 2.0367795869e-6, 2.4695242661e-6];

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
fn point(p: Pose, v: DVec3) -> DVec3 {
    DVec3::from_array(p.translation_m) + rotation(p) * v
}
fn frame(body: Pose, world: DVec3, q: DQuat) -> Pose {
    let inverse = rotation(body).conjugate();
    pose(
        inverse * (world - DVec3::from_array(body.translation_m)),
        inverse * q,
    )
}
fn body(
    mass: Option<f64>,
    half: [f64; 3],
    principal: Option<[f64; 3]>,
    guided: bool,
    lateral_free: bool,
) -> Result<CookedRigidBody, &'static str> {
    let kind = if let Some(m) = mass {
        let [x, y, z] = half;
        BodyKind::Dynamic(Inertial::new(
            m,
            principal.unwrap_or_else(|| {
                [y * y + z * z, x * x + z * z, x * x + y * y].map(|a| m * a / 3.)
            }),
            Pose::default(),
        )?)
    } else {
        BodyKind::Fixed
    };
    let source = RigidBodyDescription::new(
        kind,
        vec![ColliderDescription::new(
            ShapeDescription::Cuboid {
                half_extents_m: half,
            },
            Pose::default(),
            ContactMaterial {
                friction: 0.,
                restitution: 0.,
                memberships: 1,
                filter: 0,
            },
        )?],
    )?;
    let source = if guided {
        source.with_policy(BodyPolicy {
            enabled_translations: [true, lateral_free, true],
            enabled_rotations: [false; 3],
            can_sleep: false,
            ..Default::default()
        })?
    } else {
        source
    };
    CookedRigidBody::new(source)
}

/// Actual runtime truth sampled after a completed explicit tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Completed time, seconds.
    pub time_s: f64,
    /// Damper eye spacing, metres.
    pub length_m: f64,
    /// Slider speed, m/s; negative is compression.
    pub speed_m_s: f64,
    /// Moving carrier COM height, metres.
    pub height_m: f64,
    /// Carrier rotation relative to the frame, radians; a parallelogram preserves it.
    pub carrier_tilt_rad: f64,
    /// Largest revolute anchor gap, metres, including both loop closures.
    pub closure_error_m: f64,
    /// Difference between slider coordinate and actual eye distance, metres.
    pub slider_error_m: f64,
    /// Actual parent/child endpoint positions for plotting without a second plant.
    pub points_m: [[f64; 3]; 6],
    /// Solved fixed-frame/rocker/rocker/carrier/barrel/rod body frames, in key order.
    pub body_poses: [Pose; 6],
}

/// Installed five-body suspension and seven closure joints attached to one root.
/// Owns bindings only; the caller owns the sole runtime and clock.
pub struct Linkage {
    carrier: BodyBinding,
    slider: JointBinding,
    hinges: Vec<(BodyBinding, BodyBinding, JointDescription)>,
    rockers: [BodyBinding; 2],
    barrel: BodyBinding,
    rod: BodyBinding,
    bodies: [BodyBinding; 6],
    root_from_mount: Option<Pose>,
}
impl Linkage {
    /// Attach the source-derived linkage to an existing free chassis body.
    /// The placement maps the original bench frame into the common world. Namespaces
    /// reserve body/joint keys namespace+1..=namespace+7 (root key +1 is unused).
    /// Setup is not transactional; discard the fixture runtime after a setup error.
    pub fn attach(
        runtime: &mut PhysicalRuntime,
        parameters: Parameters,
        placement: Pose,
        root: BodyBinding,
        namespace: u64,
    ) -> Result<Self, &'static str> {
        Self::install(
            runtime,
            parameters,
            placement,
            Some(root),
            namespace,
            None,
            false,
        )
    }
    fn install(
        runtime: &mut PhysicalRuntime,
        parameters: Parameters,
        placement: Pose,
        root: Option<BodyBinding>,
        namespace: u64,
        sprung_mass_kg: Option<f64>,
        lateral_free: bool,
    ) -> Result<Self, &'static str> {
        parameters.validate()?;
        namespace
            .checked_add(7)
            .ok_or("fixture namespace overflow")?;
        let arm = DVec3::from_array(ARM);
        let second = DVec3::from_array(SECOND);
        let s = ORIGIN + DVec3::from_array(MOVING_EYE);
        let f = ORIGIN + DVec3::from_array(FIXED_EYE);
        let direction = (f - s).normalize();
        let dq = DQuat::from_rotation_arc(DVec3::X, direction);
        // Native local +X follows end 0 -> end 25 mm; local +Z is the hinge axis.
        // Body origins are the verified native COMs, not the hole centers.
        let ax = -arm.normalize();
        let az = DVec3::Y;
        let aq = DQuat::from_mat3(&DMat3::from_cols(ax, az.cross(ax), az));
        let poses = [
            pose(ORIGIN, DQuat::IDENTITY),
            pose(ORIGIN + arm * 0.5, aq),
            pose(ORIGIN + second + arm * 0.5, aq),
            pose(ORIGIN + arm, DQuat::IDENTITY),
            pose(s, dq),
            pose(f, dq),
        ];
        let mut poses = poses.map(|p| {
            pose(
                point(placement, DVec3::from_array(p.translation_m)),
                rotation(placement) * rotation(p),
            )
        });
        let root_from_mount = if let Some(root) = root {
            let actual = runtime.body_state(root)?.pose;
            let relative = frame(
                actual,
                DVec3::from_array(poses[0].translation_m),
                rotation(poses[0]),
            );
            poses[0] = actual;
            Some(relative)
        } else {
            None
        };
        // 44 g is the original assembly declaration. The 30/14 g partition and
        // non-rocker box inertias are fixture assumptions; no root CAD tensor used.
        let masses = [
            sprung_mass_kg,
            Some(ARM_MASS_KG),
            Some(ARM_MASS_KG),
            Some(parameters.load_kg),
            Some(parameters.damper_mass_kg * (30. / 44.)),
            Some(parameters.damper_mass_kg * (14. / 44.)),
        ];
        let half = [
            [0.04, 0.01, 0.005],
            [0.0225, 0.010, 0.004],
            [0.0225, 0.010, 0.004],
            [0.03, 0.02, 0.015],
            [0.025, 0.008, 0.008],
            [0.015, 0.003, 0.003],
        ];
        let mut installed = Vec::new();
        for i in 0..6 {
            if i == 0
                && let Some(root) = root
            {
                installed.push(root);
                continue;
            }
            installed.push(runtime.install_body(
                BodyKey(namespace + i as u64 + 1),
                &body(
                    masses[i],
                    half[i],
                    (i == 1 || i == 2).then_some(ARM_INERTIA_KG_M2),
                    i == 0 && sprung_mass_kg.is_some(),
                    lateral_free,
                )?,
                BodyState::at(poses[i]),
            )?);
        }
        let mut hinges = Vec::new();
        let hq = rotation(placement) * DQuat::from_rotation_arc(DVec3::X, DVec3::Y);
        for (index, (a, b, anchor)) in [
            (0, 1, ORIGIN),
            (1, 3, ORIGIN + arm),
            (0, 2, ORIGIN + second),
            (2, 3, ORIGIN + second + arm),
            (3, 4, s),
            (0, 5, f),
        ]
        .into_iter()
        .enumerate()
        {
            let anchor = point(placement, anchor);
            let desc = JointDescription::new(
                JointKind::Revolute,
                frame(poses[a], anchor, hq),
                frame(poses[b], anchor, hq),
                None,
                false,
            )?;
            runtime.install_joint(
                JointKey(namespace + index as u64 + 1),
                installed[a],
                installed[b],
                &desc,
            )?;
            hinges.push((installed[a], installed[b], desc));
        }
        let desc = JointDescription::new(
            JointKind::Prismatic,
            Pose::default(),
            Pose::default(),
            Some(JointLimits::new(
                parameters.free_length_m - parameters.stroke_m,
                parameters.free_length_m,
            )?),
            false,
        )?;
        let slider =
            runtime.install_joint(JointKey(namespace + 7), installed[4], installed[5], &desc)?;
        runtime.set_joint_motor(
            slider,
            Some(JointMotor::new(
                parameters.free_length_m,
                0.,
                parameters.stiffness_n_m,
                parameters.damping_n_s_m,
                10_000.,
            )?),
        )?;
        Ok(Self {
            carrier: installed[3],
            slider,
            hinges,
            rockers: [installed[1], installed[2]],
            barrel: installed[4],
            rod: installed[5],
            bodies: installed
                .as_slice()
                .try_into()
                .expect("six module bindings"),
            root_from_mount,
        })
    }
    /// Carrier binding for installing a real driven wheel into the same world.
    pub fn carrier(&self) -> BodyBinding {
        self.carrier
    }
    /// Capture solved frames and residuals with the caller's explicit time, seconds.
    pub fn sample(&self, runtime: &PhysicalRuntime, time_s: f64) -> Result<Sample, &'static str> {
        let state = runtime.joint_state(self.slider)?;
        let carrier = runtime.body_state(self.carrier)?.pose;
        let mut gap: f64 = 0.;
        for (a, b, j) in &self.hinges {
            let pa = runtime.body_state(*a)?.pose;
            let pb = runtime.body_state(*b)?.pose;
            let x = point(pa, DVec3::from_array(j.parent_frame().translation_m));
            let y = point(pb, DVec3::from_array(j.child_frame().translation_m));
            gap = gap.max(x.distance(y));
        }
        let c = DVec3::from_array(carrier.translation_m);
        let q = rotation(carrier);
        let actual_base = runtime.body_state(self.bodies[0])?.pose;
        let base = self.root_from_mount.map_or(actual_base, |mount| {
            pose(
                point(actual_base, DVec3::from_array(mount.translation_m)),
                rotation(actual_base) * rotation(mount),
            )
        });
        let r0 = runtime.body_state(self.rockers[0])?.pose;
        let r1 = runtime.body_state(self.rockers[1])?.pose;
        let a = point(r0, DVec3::new(-0.0125, 0., 0.));
        let b = point(r1, DVec3::new(-0.0125, 0., 0.));
        let s = DVec3::from_array(runtime.body_state(self.barrel)?.pose.translation_m);
        let f = DVec3::from_array(runtime.body_state(self.rod)?.pose.translation_m);
        let mut body_poses = [Pose::default(); 6];
        for (pose, binding) in body_poses.iter_mut().zip(self.bodies) {
            *pose = runtime.body_state(binding)?.pose;
        }
        Ok(Sample {
            time_s,
            length_m: state.position,
            speed_m_s: state.velocity,
            height_m: c.z,
            carrier_tilt_rad: q.angle_between(rotation(base)),
            closure_error_m: gap,
            slider_error_m: (s.distance(f) - state.position).abs(),
            body_poses,
            points_m: [
                point(base, DVec3::ZERO).to_array(),
                point(base, DVec3::from_array(SECOND)).to_array(),
                a.to_array(),
                b.to_array(),
                s.to_array(),
                f.to_array(),
            ],
        })
    }
}

/// Six-body, seven-joint bench. Both rockers and the damper loop are enforced by
/// runtime joints; no body pose is prescribed after installation. Linkage
/// collisions are disabled and gravity is enabled. The default frame is fixed;
/// [`Self::guided_mount`] optionally installs a guided dynamic frame.
pub struct Fixture {
    /// Sole native world, exposed for explicit diagnostics and stale-token tests.
    pub runtime: PhysicalRuntime,
    /// Approved fixture choices, retained for reset.
    pub parameters: Parameters,
    duration: StepDuration,
    tick: u64,
    linkage: Linkage,
    placement: Pose,
    sprung_mass_kg: Option<f64>,
    lateral_free: bool,
}
impl Fixture {
    /// Prepare and install a closed loop at one explicit immutable rate.
    pub fn new(parameters: Parameters, duration: StepDuration) -> Result<Self, &'static str> {
        Self::with_solver_iterations(parameters, duration, 32)
    }
    /// Explicit numerical solver budget for loop-accuracy experiments.
    pub fn with_solver_iterations(
        parameters: Parameters,
        duration: StepDuration,
        iterations: usize,
    ) -> Result<Self, &'static str> {
        Self::install(
            parameters,
            duration,
            iterations,
            Pose::default(),
            None,
            false,
        )
    }
    /// Install the same linkage on a dynamic frame guided along world X/Z.
    /// The guide locks lateral translation and all frame rotation; vertical
    /// position remains dynamic. Mass and installation orientation are explicit.
    pub fn guided_mount(
        parameters: Parameters,
        duration: StepDuration,
        iterations: usize,
        placement: Pose,
        sprung_mass_kg: f64,
    ) -> Result<Self, &'static str> {
        Self::guided_mount_with_lateral(
            parameters,
            duration,
            iterations,
            placement,
            sprung_mass_kg,
            false,
        )
    }
    /// Same test stand with optional free lateral frame translation, for omni tests.
    /// Rotation remains guided; no lateral velocity or position is prescribed.
    pub fn guided_mount_with_lateral(
        parameters: Parameters,
        duration: StepDuration,
        iterations: usize,
        placement: Pose,
        sprung_mass_kg: f64,
        lateral_free: bool,
    ) -> Result<Self, &'static str> {
        if !sprung_mass_kg.is_finite() || sprung_mass_kg <= 0. {
            return Err("invalid sprung mass");
        }
        Self::install(
            parameters,
            duration,
            iterations,
            placement,
            Some(sprung_mass_kg),
            lateral_free,
        )
    }
    fn install(
        parameters: Parameters,
        duration: StepDuration,
        iterations: usize,
        placement: Pose,
        sprung_mass_kg: Option<f64>,
        lateral_free: bool,
    ) -> Result<Self, &'static str> {
        parameters.validate()?;
        let mut runtime = PhysicalRuntime::with_solver_iterations(duration, iterations)?;
        let linkage = Linkage::install(
            &mut runtime,
            parameters,
            placement,
            None,
            0,
            sprung_mass_kg,
            lateral_free,
        )?;
        Ok(Self {
            runtime,
            parameters,
            duration,
            tick: 0,
            linkage,
            placement,
            sprung_mass_kg,
            lateral_free,
        })
    }
    /// Replace a held vertical force at the moving test load's COM, N. Zero
    /// releases the external load; gravity and the passive spring remain active.
    pub fn load(&mut self, vertical_n: f64) -> Result<(), &'static str> {
        self.runtime
            .set_body_wrench(self.linkage.carrier, [0., 0., vertical_n], [0.; 3])
    }
    /// Change damping for comparison without rebuilding or moving bodies.
    pub fn damping(&mut self, value: f64) -> Result<(), &'static str> {
        if !value.is_finite() || value < 0. {
            return Err("invalid damping");
        }
        self.runtime.set_joint_motor(
            self.linkage.slider,
            Some(JointMotor::new(
                self.parameters.free_length_m,
                0.,
                self.parameters.stiffness_n_m,
                value,
                10_000.,
            )?),
        )?;
        self.parameters.damping_n_s_m = value;
        Ok(())
    }
    /// Advance exactly one tick; no host time or prescribed mechanism update.
    pub fn step(&mut self) -> Result<Sample, &'static str> {
        let prev = self
            .tick
            .checked_mul(self.duration.as_ns())
            .ok_or("fixture time overflow")?;
        self.runtime.step(prev)?;
        self.tick += 1;
        self.sample()
    }
    /// Reconstruct the same scenario; all old bindings become stale.
    pub fn reset(&mut self) -> Result<(), &'static str> {
        *self = Self::install(
            self.parameters,
            self.duration,
            self.runtime.solver_iterations(),
            self.placement,
            self.sprung_mass_kg,
            self.lateral_free,
        )?;
        Ok(())
    }
    /// Read the slider binding for stale-token validation; never a serialized ID.
    pub fn slider(&self) -> JointBinding {
        self.linkage.slider
    }
    /// Capture solved poses and loop residuals without allocating mesh data.
    pub fn sample(&self) -> Result<Sample, &'static str> {
        self.linkage.sample(
            &self.runtime,
            self.tick as f64 * self.duration.as_secs_f64(),
        )
    }
}

fn write_sample(csv: &mut impl std::io::Write, x: Sample) -> std::io::Result<()> {
    let points = x.points_m.map(|v| format!(",{},{}", v[0], v[2])).concat();
    let poses = x
        .body_poses
        .iter()
        .flat_map(|p| p.translation_m.into_iter().chain(p.rotation_wxyz))
        .map(|v| format!(",{v}"))
        .collect::<String>();
    writeln!(
        csv,
        "{},{},{},{},{},{},{}{}{poses}",
        x.time_s,
        x.length_m,
        x.speed_m_s,
        x.height_m,
        x.carrier_tilt_rad,
        x.closure_error_m,
        x.slider_error_m,
        points
    )?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let output = args.next().unwrap_or_else(|| "suspension.csv".into());
    let mut parameters = Parameters::default();
    if let Some(d) = args.next() {
        parameters.damping_n_s_m = d.parse()?;
    }
    let iterations = args.next().map(|x| x.parse()).transpose()?.unwrap_or(32);
    let hz = args.next().map(|x| x.parse()).transpose()?.unwrap_or(1000);
    let duration = StepDuration::from_hz(hz)?;
    let mut fixture = Fixture::with_solver_iterations(parameters, duration, iterations)?;
    let axial_test_force: Option<f64> = args.next().map(|x| x.parse()).transpose()?;
    if let Some(force) = axial_test_force {
        fixture.runtime.set_joint_effort(fixture.slider(), force)?;
    }
    use std::io::Write;
    let mut csv = std::io::BufWriter::new(std::fs::File::create(output)?);
    let pose_header = (1..=6)
        .flat_map(|i| ["x", "y", "z", "qw", "qx", "qy", "qz"].map(move |k| format!(",body{i}_{k}")))
        .collect::<String>();
    writeln!(
        csv,
        "time_s,length_m,speed_m_s,height_m,tilt_rad,closure_error_m,slider_error_m,ax,az,bx,bz,cx,cz,dx,dz,sx,sz,fx,fz{pose_header}"
    )?;
    write_sample(&mut csv, fixture.sample()?)?;
    let ticks = 8_000_000_000 / duration.as_ns();
    let pulse_start = 3_000_000_000 / duration.as_ns();
    let pulse_end = 3_250_000_000 / duration.as_ns();
    let started = std::time::Instant::now();
    for tick in 0..ticks {
        // Hold -20 N from 3–3.25 s, then observe passive recovery.
        if axial_test_force.is_none() && tick == pulse_start {
            fixture.load(-20.)?;
        } else if axial_test_force.is_none() && tick == pulse_end {
            fixture.load(0.)?;
        }
        write_sample(&mut csv, fixture.step()?)?;
    }
    eprintln!(
        "{:?}; iterations={iterations} hz={hz} elapsed={:?}; final {:?}",
        parameters,
        started.elapsed(),
        fixture.sample()?
    );
    Ok(())
}
