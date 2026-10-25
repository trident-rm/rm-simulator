// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Non-RoboMaster XY positioning robot with caller-owned feedback and CPU timing.
use sim_engine::{
    Pose,
    model::{
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, JointDescription,
        JointKind, JointLimits, JointState, RigidBodyDescription, ShapeDescription,
    },
};
use sim_physics::{
    PhysicalRuntime, StepDuration,
    cooking::CookedRigidBody,
    runtime::{BodyKey, JointBinding, JointKey, session::PhysicalSession},
};

/// Two independent joint keys for the X carriage and its Y slide.
pub const AXES: [JointKey; 2] = [JointKey(1), JointKey(2)];

/// Prepare three box links once. Dimensions and uniform-solid inertias are
/// illustrative fixture choices, not calibrated hardware or CAD properties.
pub fn prepare() -> Result<[CookedRigidBody; 3], &'static str> {
    let make = |mass_kg: Option<f64>| {
        let half = [0.1, 0.1, 0.05];
        let kind = if let Some(mass) = mass_kg {
            let [x, y, z] = half;
            let inertia = [y * y + z * z, x * x + z * z, x * x + y * y].map(|v| mass * v / 3.);
            BodyKind::Dynamic(Inertial::new(mass, inertia, Pose::default())?)
        } else {
            BodyKind::Fixed
        };
        CookedRigidBody::new(RigidBodyDescription::new(
            kind,
            vec![ColliderDescription::new(
                ShapeDescription::Cuboid {
                    half_extents_m: half,
                },
                Pose::default(),
                ContactMaterial {
                    friction: 0.4,
                    restitution: 0.,
                    memberships: u32::MAX,
                    filter: u32::MAX,
                },
            )?],
        )?)
    };
    Ok([make(None)?, make(Some(2.))?, make(Some(1.))?])
}

/// Install a fixed support and two stacked, gravity-loaded prismatic stages.
/// The X joint moves 3 kg, including the 1 kg Y stage. Both have ±0.5 m stops.
/// Cloned cooked resources share shapes; every installation has independent state.
pub fn install(
    runtime: &mut PhysicalRuntime,
    prepared: &[CookedRigidBody; 3],
) -> Result<(), &'static str> {
    let mut bodies = Vec::with_capacity(3);
    for (index, model) in prepared.iter().enumerate() {
        bodies.push(runtime.install_body(
            BodyKey(index as u64 + 1),
            model,
            BodyState::at(Pose::at([0., 0., index as f64 * 0.2])),
        )?);
    }
    for (axis, key) in AXES.into_iter().enumerate() {
        let mut parent = Pose::at([0., 0., 0.2]);
        let mut child = Pose::default();
        if axis == 1 {
            parent = Pose::yawed(parent.translation_m, std::f64::consts::FRAC_PI_2);
            child = Pose::yawed(child.translation_m, std::f64::consts::FRAC_PI_2);
        }
        runtime.install_joint(
            key,
            bodies[axis],
            bodies[axis + 1],
            &JointDescription::new(
                JointKind::Prismatic,
                parent,
                child,
                Some(JointLimits::new(-0.5, 0.5)?),
                false,
            )?,
        )?;
    }
    Ok(())
}

/// Reacquire two world-scoped bindings after construction or successful reset.
pub fn bindings(runtime: &PhysicalRuntime) -> Result<[JointBinding; 2], &'static str> {
    Ok([
        runtime.joint_binding(AXES[0]).ok_or("missing X slide")?,
        runtime.joint_binding(AXES[1]).ok_or("missing Y slide")?,
    ])
}

/// Read only compact physical truth. No geometry or checkpoint is captured.
pub fn observe(
    runtime: &PhysicalRuntime,
    joints: [JointBinding; 2],
) -> Result<[JointState; 2], &'static str> {
    Ok([
        runtime.joint_state(joints[0])?,
        runtime.joint_state(joints[1])?,
    ])
}

/// Caller-side zero-order-held PD force, computed once before every physical tick.
/// Gains are 120 N/m and 30 N s/m; each axis saturates at ±20 N.
/// This uses perfect joint truth, without an encoder or motor electrical model.
pub fn control(
    runtime: &mut PhysicalRuntime,
    joints: [JointBinding; 2],
    target_m: [f64; 2],
) -> Result<(), &'static str> {
    let states = observe(runtime, joints)?;
    for axis in 0..2 {
        let force_n = (120. * (target_m[axis] - states[axis].position)
            - 30. * states[axis].velocity)
            .clamp(-20., 20.);
        runtime.set_joint_effort(joints[axis], force_n)?;
    }
    Ok(())
}

/// Switch between two positions every two simulated seconds, independent of rate.
pub fn target(time_ns: u64) -> [f64; 2] {
    if (time_ns / 2_000_000_000).is_multiple_of(2) {
        [0.25, -0.2]
    } else {
        [-0.15, 0.3]
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::time::Instant;
    let mut args = std::env::args().skip(1);
    let hz: u64 = args.next().unwrap_or_else(|| "1000".into()).parse()?;
    if ![128, 1000].contains(&hz) {
        return Err("expected 128 or 1000 Hz".into());
    }
    let steps: u64 = args
        .next()
        .unwrap_or_else(|| (10 * hz).to_string())
        .parse()?;
    if steps == 0 || args.next().is_some() {
        return Err("usage: controller_bench [128|1000] [positive STEPS]".into());
    }
    let duration = StepDuration::from_hz(hz)?;
    // Reject an overflowing horizon before constructing or running the plant.
    steps
        .checked_mul(duration.as_ns())
        .ok_or("simulation time overflow")?;
    let start = Instant::now();
    let prepared = prepare()?;
    let recipe = || PhysicalRuntime::from_recipe(duration, |runtime| install(runtime, &prepared));
    let mut session = PhysicalSession::new(recipe)?;
    let setup = start.elapsed();
    // Warm the native solver, then restart the episode and controller bindings.
    session.step(128)?;
    let reset_start = Instant::now();
    session.reset()?;
    let reset = reset_start.elapsed();
    let joints = bindings(session.runtime())?;
    let start = Instant::now();
    for _ in 0..steps {
        let goal = target(session.stamp().time_ns);
        control(session.runtime_mut(), joints, goal)?;
        session.step(1)?;
    }
    let elapsed = start.elapsed();
    let states = observe(session.runtime(), joints)?;
    let goal = target(session.stamp().time_ns - duration.as_ns());
    println!(
        "rate_hz={hz} steps={steps} simulation_ns={}",
        session.stamp().time_ns
    );
    println!(
        "setup_ms={:.3} reset_ms={:.3} loop_ms={:.3} us_per_step={:.3} realtime_factor={:.2}",
        setup.as_secs_f64() * 1e3,
        reset.as_secs_f64() * 1e3,
        elapsed.as_secs_f64() * 1e3,
        elapsed.as_secs_f64() * 1e6 / steps as f64,
        session.stamp().time_ns as f64 * 1e-9 / elapsed.as_secs_f64()
    );
    println!(
        "target_m={goal:?} position_m={:?} error_m={:?}",
        states.map(|s| s.position),
        std::array::from_fn::<_, 2, _>(|i| goal[i] - states[i].position)
    );
    Ok(())
}
