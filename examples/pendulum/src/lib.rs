// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! An independent motor, encoder and controller composition over generic physics.
//! Parameters are illustrative. Controllers receive sampled encoder observations;
//! physical truth is reserved for the plant, evaluation and passive rendering.
#![deny(missing_docs)]
use sim_engine::{
    Pose,
    model::{
        AssemblyDescription, AssemblyJoint, AssemblyJointId, AssemblyLink, AssemblyLinkId,
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, JointDescription,
        JointKind, JointLimits, JointState, RigidBodyDescription, ShapeDescription,
    },
    time::Stamp,
};
use sim_physics::{
    PhysicalRuntime, StepDuration,
    cooking::CookedAssembly,
    runtime::{AssemblyKeys, BodyKey, JointBinding, JointKey, session::PhysicalSession},
};
use std::f64::consts::{FRAC_1_SQRT_2, TAU};

/// Encoder period, nanoseconds (200 Hz).
pub const ENCODER_PERIOD_NS: u64 = 5_000_000;
/// Controller period, nanoseconds (100 Hz); actuation updates at 1000 Hz.
pub const CONTROL_PERIOD_NS: u64 = 10_000_000;
/// Encoder angle represented by one count, radians (4096 counts/revolution).
pub const ENCODER_RAD_PER_COUNT: f64 = TAU / 4096.;
/// Ideal bidirectional supply voltage limit, volts.
pub const SUPPLY_V: f64 = 12.;
/// Symmetric winding current limit, amperes.
pub const CURRENT_LIMIT_A: f64 = 3.;
/// Source hinge key; its positive axis is world +Y.
pub const HINGE: JointKey = JointKey(1);

/// Immutable visual recipe shared by the headless model and viewer adapter.
#[derive(Clone, Copy, Debug)]
pub struct VisualPart {
    /// Body key resolved afresh after reset.
    pub body: BodyKey,
    /// Box half extents in body-local FLU metres.
    pub half_extents_m: [f64; 3],
    /// Display color in sRGB, with components in [0, 1].
    pub color_srgb: [f32; 3],
}
/// Fixed support and a 0.6 m rod; these dimensions also construct the colliders.
pub const PARTS: [VisualPart; 2] = [
    VisualPart {
        body: BodyKey(1),
        half_extents_m: [0.06, 0.12, 0.06],
        color_srgb: [0.16, 0.24, 0.32],
    },
    VisualPart {
        body: BodyKey(2),
        half_extents_m: [0.025, 0.025, 0.3],
        color_srgb: [0.98, 0.48, 0.16],
    },
];

fn prepare() -> Result<CookedAssembly, &'static str> {
    let body = |part: VisualPart, mass_kg: Option<f64>| {
        let kind = if let Some(mass) = mass_kg {
            let [x, y, z] = part.half_extents_m;
            let inertia = [y * y + z * z, x * x + z * z, x * x + y * y].map(|v| mass * v / 3.);
            BodyKind::Dynamic(Inertial::new(mass, inertia, Pose::default())?)
        } else {
            BodyKind::Fixed
        };
        RigidBodyDescription::new(
            kind,
            vec![ColliderDescription::new(
                ShapeDescription::Cuboid {
                    half_extents_m: part.half_extents_m,
                },
                Pose::default(),
                ContactMaterial {
                    friction: 0.4,
                    restitution: 0.,
                    memberships: u32::MAX,
                    filter: u32::MAX,
                },
            )?],
        )
    };
    let angle_rad: f64 = 0.55;
    let rod_pose = Pose {
        translation_m: [-0.3 * angle_rad.sin(), 0., 1.1 - 0.3 * angle_rad.cos()],
        rotation_wxyz: [(angle_rad / 2.).cos(), 0., (angle_rad / 2.).sin(), 0.],
    };
    let frame = |translation_m| Pose {
        translation_m,
        rotation_wxyz: [FRAC_1_SQRT_2, 0., 0., FRAC_1_SQRT_2],
    };
    Ok(CookedAssembly::new(AssemblyDescription::new(
        AssemblyLinkId(1),
        vec![
            AssemblyLink::new(
                AssemblyLinkId(1),
                body(PARTS[0], None)?,
                BodyState::at(Pose::at([0., 0., 1.1])),
            )?,
            AssemblyLink::new(
                AssemblyLinkId(2),
                body(PARTS[1], Some(0.5))?,
                BodyState::at(rod_pose),
            )?,
        ],
        vec![AssemblyJoint::new(
            AssemblyJointId(1),
            AssemblyLinkId(1),
            AssemblyLinkId(2),
            JointDescription::new(
                JointKind::Revolute,
                frame([0.; 3]),
                frame([0., 0., 0.3]),
                Some(JointLimits::new(-1.2, 1.2)?),
                false,
            )?,
        )],
    )?)?)
}

/// Latest held sensor observation. It contains no exact physical velocity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EncoderSample {
    /// Actual acquisition time, nanoseconds; held values retain their original stamp.
    pub time_ns: u64,
    /// Signed quantized count within the bounded hinge's principal angle range.
    pub count: i64,
}
impl EncoderSample {
    /// Measured angle, radians, derived only from the quantized count.
    pub fn angle_rad(self) -> f64 {
        self.count as f64 * ENCODER_RAD_PER_COUNT
    }
}

#[derive(Default)]
struct Encoder {
    next_ns: u64,
    latest: Option<EncoderSample>,
    samples: u64,
}
impl Encoder {
    fn acquire(&mut self, now_ns: u64, angle_rad: f64) {
        if now_ns == self.next_ns {
            self.latest = Some(EncoderSample {
                time_ns: now_ns,
                count: (angle_rad / ENCODER_RAD_PER_COUNT).round() as i64,
            });
            self.samples += 1;
            self.next_ns = self.next_ns.saturating_add(ENCODER_PERIOD_NS);
        }
    }
}

/// Held actuator output from the illustrative electrical/mechanical motor model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorOutput {
    /// Applied bridge voltage after supply saturation, volts.
    pub voltage_v: f64,
    /// Current after ideal limiting, amperes; not an emulated current sensor.
    pub current_a: f64,
    /// Applied torque after viscous loss, newton metres.
    pub torque_nm: f64,
}
/// Illustrative algebraic DC motor with R=2 Ω, Kt=0.4 N m/A,
/// Ke=0.4 V s/rad and viscous loss=0.03 N m s/rad. No inductance or bus model.
#[derive(Default)]
pub struct Motor {
    last: MotorOutput,
}
impl Motor {
    /// Resolve held voltage and actual plant speed into the next held torque.
    /// Require finite inputs before mutation. The ideal bridge can absorb braking
    /// current; no claim of hardware calibration or electrical energy storage is made.
    pub fn apply(
        &mut self,
        requested_v: f64,
        speed_rad_s: f64,
    ) -> Result<MotorOutput, &'static str> {
        if !requested_v.is_finite() || !speed_rad_s.is_finite() {
            return Err("motor inputs must be finite");
        }
        let voltage_v = requested_v.clamp(-SUPPLY_V, SUPPLY_V);
        let current_a =
            ((voltage_v - 0.4 * speed_rad_s) / 2.).clamp(-CURRENT_LIMIT_A, CURRENT_LIMIT_A);
        let torque_nm = 0.4 * current_a - 0.03 * speed_rad_s;
        if !torque_nm.is_finite() {
            return Err("motor torque overflow");
        }
        self.last = MotorOutput {
            voltage_v,
            current_a,
            torque_nm,
        };
        Ok(self.last)
    }
}

#[derive(Default)]
struct Controller {
    next_ns: u64,
    previous: Option<EncoderSample>,
    velocity_rad_s: f64,
    integral_rad_s: f64,
    voltage_v: f64,
    updates: u64,
}
impl Controller {
    // This interface deliberately accepts only the measured encoder and target.
    // No runtime or ground-truth position/velocity enters the feedback controller.
    fn update(&mut self, sample: EncoderSample, target_rad: f64) {
        let dt_s = CONTROL_PERIOD_NS as f64 * 1e-9;
        if let Some(previous) = self.previous {
            let measured_dt_s = (sample.time_ns - previous.time_ns) as f64 * 1e-9;
            let speed = (sample.angle_rad() - previous.angle_rad()) / measured_dt_s;
            self.velocity_rad_s += dt_s / (0.03 + dt_s) * (speed - self.velocity_rad_s);
        }
        let error_rad = target_rad - sample.angle_rad();
        let next_integral = self.integral_rad_s + error_rad * dt_s;
        let candidate_v = 20. * error_rad - 4. * self.velocity_rad_s + 24. * next_integral;
        // Limit integral growth during voltage saturation; current limiting remains in the plant.
        if candidate_v.abs() <= SUPPLY_V || candidate_v * error_rad < 0. {
            self.integral_rad_s = next_integral;
        }
        self.voltage_v = (20. * error_rad - 4. * self.velocity_rad_s + 24. * self.integral_rad_s)
            .clamp(-SUPPLY_V, SUPPLY_V);
        self.previous = Some(sample);
        self.updates += 1;
        self.next_ns = self.next_ns.saturating_add(CONTROL_PERIOD_NS);
    }
}

/// Alternate between two references every four simulated seconds, in radians.
pub fn reference_rad(time_ns: u64) -> f64 {
    if (time_ns / 4_000_000_000).is_multiple_of(2) {
        0.35
    } else {
        -0.35
    }
}

/// Compact extension telemetry. Exact physical state is accessed separately.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Telemetry {
    /// Latest acquired encoder sample; absent immediately after reset.
    pub encoder: Option<EncoderSample>,
    /// Number of actual encoder acquisitions since reset.
    pub encoder_samples: u64,
    /// Number of actual feedback controller updates since reset.
    pub controller_updates: u64,
    /// Last held actuator output, resolved at the preceding tick's start.
    pub motor: MotorOutput,
}

type Recipe =
    Box<dyn Fn() -> Result<PhysicalRuntime, sim_physics::runtime::RuntimeError> + Send + Sync>;

/// One caller-paced experiment, with explicit extension-owned sensor/control state.
/// One boxed setup recipe retains cooked resources for reconstruction; it is
/// invoked only on construction/reset. The control loop uses ordinary typed calls.
///
/// ```
/// use sim_pendulum_example::Experiment;
/// let mut experiment = Experiment::new()?;
/// experiment.advance(1000)?;
/// assert_eq!(experiment.stamp().time_ns, 1_000_000_000);
/// assert_eq!(experiment.telemetry().encoder_samples, 200);
/// experiment.reset()?;
/// assert!(experiment.telemetry().encoder.is_none());
/// # Ok::<(), &'static str>(())
/// ```
pub struct Experiment {
    session: PhysicalSession<Recipe>,
    hinge: JointBinding,
    encoder: Encoder,
    controller: Controller,
    motor: Motor,
}
impl Experiment {
    /// Prepare the illustrated model once and install a 1000 Hz physical scene.
    pub fn new() -> Result<Self, &'static str> {
        let model = prepare()?;
        let keys = AssemblyKeys {
            links: vec![
                (AssemblyLinkId(1), PARTS[0].body),
                (AssemblyLinkId(2), PARTS[1].body),
            ],
            joints: vec![(AssemblyJointId(1), HINGE)],
        };
        let recipe: Recipe = Box::new(move || {
            PhysicalRuntime::from_recipe(StepDuration::HZ_1000, |runtime| {
                runtime.install_assembly(&model, Pose::default(), &keys)?;
                Ok(())
            })
        });
        let session = PhysicalSession::new(recipe)?;
        let hinge = session
            .runtime()
            .joint_binding(HINGE)
            .ok_or("missing pendulum hinge")?;
        Ok(Self {
            session,
            hinge,
            encoder: Encoder::default(),
            controller: Controller::default(),
            motor: Motor::default(),
        })
    }
    /// Completed physical time; sampling schedules are independent extension state.
    pub fn stamp(&self) -> Stamp {
        self.session.stamp()
    }
    /// Borrow privileged runtime truth for rendering, diagnostics and binding tests.
    pub fn runtime(&self) -> &PhysicalRuntime {
        self.session.runtime()
    }
    /// Exact joint truth for evaluation; the controller does not consume this API.
    pub fn truth(&self) -> Result<JointState, &'static str> {
        Ok(self.runtime().joint_state(self.hinge)?)
    }
    /// Compact sensor/control/actuation telemetry, with no geometry capture.
    pub fn telemetry(&self) -> Telemetry {
        Telemetry {
            encoder: self.encoder.latest,
            encoder_samples: self.encoder.samples,
            controller_updates: self.controller.updates,
            motor: self.motor.last,
        }
    }
    /// Advance explicit ticks with 200 Hz sampling, 100 Hz feedback and 1000 Hz
    /// motor/physics updates. Completed earlier ticks remain if a step fails.
    /// Samples use actual tick-start state; no historical samples are synthesized.
    pub fn advance(&mut self, steps: u64) -> Result<(), &'static str> {
        let stamp = self.stamp();
        let elapsed_ns = steps
            .checked_mul(StepDuration::HZ_1000.as_ns())
            .ok_or("experiment time overflow")?;
        stamp
            .time_ns
            .checked_add(elapsed_ns)
            .ok_or("experiment time overflow")?;
        stamp
            .tick
            .checked_add(steps)
            .ok_or("experiment tick overflow")?;
        for _ in 0..steps {
            let now_ns = self.stamp().time_ns;
            let truth = self.truth()?;
            self.encoder.acquire(now_ns, truth.position);
            if now_ns == self.controller.next_ns {
                self.controller.update(
                    self.encoder.latest.ok_or("controller needs encoder")?,
                    reference_rad(now_ns),
                );
            }
            let output = self
                .motor
                .apply(self.controller.voltage_v, truth.velocity)?;
            self.session
                .runtime_mut()
                .set_joint_effort(self.hinge, output.torque_nm)?;
            self.session.step(1)?;
        }
        Ok(())
    }
    /// Reconstruct from shared prepared resources, reacquire the hinge and clear
    /// motor output, encoder hold/phase, controller history, integral and voltage.
    pub fn reset(&mut self) -> Result<(), &'static str> {
        self.session.reset()?;
        self.hinge = self
            .runtime()
            .joint_binding(HINGE)
            .ok_or("missing reset hinge")?;
        self.encoder = Encoder::default();
        self.controller = Controller::default();
        self.motor = Motor::default();
        Ok(())
    }
}
