// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Standalone physical yaw/pitch fixture; no game controller or CAD is involved.
use rm_sim_physics::{
    StepDuration,
    cooking::CookedAssembly,
    runtime::{AssemblyKeys, BodyKey, JointKey},
    session::{RobotScenario, RobotSession},
};
use sim_engine::{
    Pose,
    model::{
        AssemblyDescription, AssemblyJoint, AssemblyJointId, AssemblyLink, AssemblyLinkId,
        BodyKind, BodyState, ColliderDescription, ContactMaterial, Inertial, JointDescription,
        JointKind, JointLimits, JointMotor, RigidBodyDescription, ShapeDescription,
    },
};

/// Logical pedestal body key; bindings must be reacquired after recipe reset.
pub const BASE: BodyKey = BodyKey(1);
/// Logical yaw yoke body key.
pub const YOKE: BodyKey = BodyKey(2);
/// Logical pitch barrel body key.
pub const BARREL: BodyKey = BodyKey(3);
/// Yaw joint about the pedestal's local +Z, in radians.
pub const YAW: JointKey = JointKey(1);
/// Pitch joint about the yoke's local +Y, in radians.
pub const PITCH: JointKey = JointKey(2);
/// Symmetric yaw hard stop, radians; a fixture choice, not a competition rule.
pub const YAW_LIMIT_RAD: f64 = 1.;
/// Symmetric pitch hard stop, radians; a fixture choice, not a competition rule.
pub const PITCH_LIMIT_RAD: f64 = 0.45;

/// Source pedestal identity, independent of a world's body keys.
pub const BASE_LINK: AssemblyLinkId = AssemblyLinkId(101);
/// Source yoke identity.
pub const YOKE_LINK: AssemblyLinkId = AssemblyLinkId(102);
/// Source barrel identity.
pub const BARREL_LINK: AssemblyLinkId = AssemblyLinkId(103);
/// Source yaw connection identity.
pub const YAW_JOINT: AssemblyJointId = AssemblyJointId(101);
/// Source pitch connection identity.
pub const PITCH_JOINT: AssemblyJointId = AssemblyJointId(102);

/// Map the reusable source identities to this fixture's original runtime keys.
pub fn keys() -> AssemblyKeys {
    AssemblyKeys {
        links: vec![(BASE_LINK, BASE), (YOKE_LINK, YOKE), (BARREL_LINK, BARREL)],
        joints: vec![(YAW_JOINT, YAW), (PITCH_JOINT, PITCH)],
    }
}

fn cuboid(mass_kg: Option<f64>, half: [f64; 3]) -> Result<RigidBodyDescription, &'static str> {
    let kind = if let Some(mass) = mass_kg {
        // Uniform solid box, principal inertia at its explicit central COM.
        let [x, y, z] = half;
        let inertia = [y * y + z * z, x * x + z * z, x * x + y * y].map(|v| mass * v / 3.);
        BodyKind::Dynamic(Inertial::new(mass, inertia, Pose::default())?)
    } else {
        BodyKind::Fixed
    };
    RigidBodyDescription::new(
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
    )
}

/// Three-link source in shared assembly-local FLU, with its base at z = 1 m.
/// A dynamic pedestal permits reaction/contact tests; the example fixes it.
/// Dimensions and inertial properties are illustrative, not source CAD values.
pub fn description(dynamic_base: bool) -> Result<AssemblyDescription, &'static str> {
    let base = cuboid(dynamic_base.then_some(10.), [0.25, 0.25, 0.1])?;
    let yoke = cuboid(Some(2.), [0.12, 0.12, 0.1])?;
    let barrel = cuboid(Some(1.), [0.3, 0.05, 0.05])?;
    // Joint-local +X is the degree of freedom. Rotate it onto body +Z/+Y.
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let yaw_frame = |translation_m| Pose {
        translation_m,
        rotation_wxyz: [h, 0., -h, 0.],
    };
    let pitch_frame = |translation_m| Pose {
        translation_m,
        rotation_wxyz: [h, 0., 0., h],
    };
    let yaw = JointDescription::new(
        JointKind::Revolute,
        yaw_frame([0., 0., 0.1]),
        yaw_frame([0., 0., -0.1]),
        Some(JointLimits::new(-YAW_LIMIT_RAD, YAW_LIMIT_RAD)?),
        false,
    )?;
    let pitch = JointDescription::new(
        JointKind::Revolute,
        pitch_frame([0., 0., 0.2]),
        pitch_frame([-0.25, 0., 0.]),
        Some(JointLimits::new(-PITCH_LIMIT_RAD, PITCH_LIMIT_RAD)?),
        false,
    )?;
    AssemblyDescription::new(
        BASE_LINK,
        vec![
            AssemblyLink::new(BASE_LINK, base, BodyState::at(Pose::at([0., 0., 1.])))?,
            AssemblyLink::new(YOKE_LINK, yoke, BodyState::at(Pose::at([0., 0., 1.2])))?,
            AssemblyLink::new(
                BARREL_LINK,
                barrel,
                BodyState::at(Pose::at([0.25, 0., 1.4])),
            )?,
        ],
        vec![
            AssemblyJoint::new(YAW_JOINT, BASE_LINK, YOKE_LINK, yaw),
            AssemblyJoint::new(PITCH_JOINT, YOKE_LINK, BARREL_LINK, pitch),
        ],
    )
}

/// Repeatable three-body fixture at 1000 Hz, with a floor at zero metres.
/// Prepare once; construction and reset install the same immutable resource.
pub fn scenario(dynamic_base: bool) -> Result<RobotScenario, &'static str> {
    let prepared = CookedAssembly::new(description(dynamic_base)?)?;
    let keys = keys();
    RobotScenario::new(vec![], 0., StepDuration::HZ_1000, move |world| {
        world.install_assembly(&prepared, Pose::default(), &keys)?;
        Ok(())
    })
}

/// Set both held position motors, with zero target velocity and bounded torque.
/// Targets use radians; gains are 200 N m/rad and 10 N m s/rad, capped at 20 N m.
pub fn aim(session: &mut RobotSession, yaw_rad: f64, pitch_rad: f64) -> Result<(), &'static str> {
    for (key, target) in [(YAW, yaw_rad), (PITCH, pitch_rad)] {
        let binding = session
            .ground_truth()
            .joint_binding(key)
            .ok_or("missing fixture joint")?;
        session
            .physics_mut()
            .set_joint_motor(binding, Some(JointMotor::new(target, 0., 200., 10., 20.)?))?;
    }
    Ok(())
}

fn main() -> Result<(), &'static str> {
    let mut session = RobotSession::new(scenario(false)?)?;
    aim(&mut session, 0.6, -0.25)?;
    let mut contacts = Vec::new();
    let start = std::time::Instant::now();
    session.step_into(2000, &mut contacts)?;
    println!(
        "2000 fixture ticks (1000 Hz): {:?} wall time",
        start.elapsed()
    );
    for (key, target_rad) in [(YAW, 0.6), (PITCH, -0.25)] {
        let binding = session
            .ground_truth()
            .joint_binding(key)
            .ok_or("missing fixture joint")?;
        let state = session.ground_truth().joint_state(binding)?;
        println!("{key:?}: {state:?}");
        assert!((state.position - target_rad).abs() < 0.025);
        session.physics_mut().set_joint_motor(binding, None)?;
        session.physics_mut().set_joint_effort(binding, 5.)?;
    }
    session.step_into(2000, &mut contacts)?;
    for (key, limit_rad) in [(YAW, YAW_LIMIT_RAD), (PITCH, PITCH_LIMIT_RAD)] {
        let binding = session
            .ground_truth()
            .joint_binding(key)
            .ok_or("missing fixture joint")?;
        let state = session.ground_truth().joint_state(binding)?;
        println!("{key:?} under +5 N m: {state:?}");
        assert!((state.position - limit_rad).abs() < 0.025);
    }
    let old = session
        .ground_truth()
        .joint_binding(PITCH)
        .ok_or("missing pitch")?;
    session.reset()?;
    assert!(session.ground_truth().joint_state(old).is_err());
    let rebound = session
        .ground_truth()
        .joint_binding(PITCH)
        .ok_or("missing reset pitch")?;
    assert_ne!(old, rebound);
    println!(
        "recipe reset pitch: {:?}",
        session.ground_truth().joint_state(rebound)?
    );
    Ok(())
}
